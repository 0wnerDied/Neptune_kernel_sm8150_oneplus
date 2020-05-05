// SPDX-License-Identifier: GPL-2.0
/*
 * Encryption policy functions for per-file encryption support.
 *
 * Copyright (C) 2015, Google, Inc.
 * Copyright (C) 2015, Motorola Mobility.
 *
 * Written by Michael Halcrow, 2015.
 * Modified by Jaegeuk Kim, 2015.
 */

#include <linux/random.h>
#include <linux/string.h>
#include <linux/mount.h>
#include "fscrypt_private.h"

/*
 * check whether an encryption policy is consistent with an encryption context
 */
static bool is_encryption_context_consistent_with_policy(
				const struct fscrypt_context *ctx,
				const struct fscrypt_policy *policy)
{
	if (policy1->version != policy2->version)
		return false;

	return !memcmp(policy1, policy2, fscrypt_policy_size(policy1));
}

static bool fscrypt_valid_enc_modes(u32 contents_mode, u32 filenames_mode)
{
	if (contents_mode == FSCRYPT_MODE_AES_256_XTS &&
	    filenames_mode == FSCRYPT_MODE_AES_256_CTS)
		return true;

	if (contents_mode == FSCRYPT_MODE_AES_128_CBC &&
	    filenames_mode == FSCRYPT_MODE_AES_128_CTS)
		return true;

	if (contents_mode == FSCRYPT_MODE_ADIANTUM &&
	    filenames_mode == FSCRYPT_MODE_ADIANTUM)
		return true;

	if (contents_mode == FSCRYPT_MODE_PRIVATE &&
	    filenames_mode == FSCRYPT_MODE_AES_256_CTS)
		return true;

	return false;
}

static int create_encryption_context_from_policy(struct inode *inode,
				const struct fscrypt_policy *policy)
{
	struct fscrypt_context ctx;

	ctx.format = FS_ENCRYPTION_CONTEXT_FORMAT_V1;
	memcpy(ctx.master_key_descriptor, policy->master_key_descriptor,
					FS_KEY_DESCRIPTOR_SIZE);

	if (!fscrypt_valid_enc_modes(policy->contents_encryption_mode,
				     policy->filenames_encryption_mode)) {
		fscrypt_warn(inode,
			     "Unsupported encryption modes (contents %d, filenames %d)",
			     policy->contents_encryption_mode,
			     policy->filenames_encryption_mode);
		return false;
	}

	if (policy->contents_encryption_mode == FSCRYPT_MODE_PRIVATE) {
		fscrypt_warn(inode,
			     "FSCRYPT_MODE_PRIVATE is only supported for v1 policies");
		return false;
	}

	if (policy->flags & ~FSCRYPT_POLICY_FLAGS_VALID) {
		fscrypt_warn(inode, "Unsupported encryption flags (0x%02x)",
			     policy->flags);
		return false;
	}

	if ((policy->flags & FSCRYPT_POLICY_FLAG_DIRECT_KEY) &&
	    !supported_direct_key_modes(inode, policy->contents_encryption_mode,
					policy->filenames_encryption_mode))
		return false;

	if ((policy->flags & FSCRYPT_POLICY_FLAG_IV_INO_LBLK_64) &&
	    !supported_iv_ino_lblk_64_policy(policy, inode))
		return false;

	if (memchr_inv(policy->__reserved, 0, sizeof(policy->__reserved))) {
		fscrypt_warn(inode, "Reserved bits set in encryption policy");
		return false;
	}

	return true;
}

/**
 * fscrypt_supported_policy - check whether an encryption policy is supported
 *
 * Given an encryption policy, check whether all its encryption modes and other
 * settings are supported by this kernel on the given inode.  (But we don't
 * currently don't check for crypto API support here, so attempting to use an
 * algorithm not configured into the crypto API will still fail later.)
 *
 * Return: %true if supported, else %false
 */
bool fscrypt_supported_policy(const union fscrypt_policy *policy_u,
			      const struct inode *inode)
{
	switch (policy_u->version) {
	case FSCRYPT_POLICY_V1:
		return fscrypt_supported_v1_policy(&policy_u->v1, inode);
	case FSCRYPT_POLICY_V2:
		return fscrypt_supported_v2_policy(&policy_u->v2, inode);
	}
	return false;
}

/**
 * fscrypt_new_context_from_policy - create a new fscrypt_context from a policy
 *
 * Create an fscrypt_context for an inode that is being assigned the given
 * encryption policy.  A new nonce is randomly generated.
 *
 * Return: the size of the new context in bytes.
 */
static int fscrypt_new_context_from_policy(union fscrypt_context *ctx_u,
					   const union fscrypt_policy *policy_u)
{
	memset(ctx_u, 0, sizeof(*ctx_u));

	switch (policy_u->version) {
	case FSCRYPT_POLICY_V1: {
		const struct fscrypt_policy_v1 *policy = &policy_u->v1;
		struct fscrypt_context_v1 *ctx = &ctx_u->v1;

		ctx->version = FSCRYPT_CONTEXT_V1;
		ctx->contents_encryption_mode =
			policy->contents_encryption_mode;
		ctx->filenames_encryption_mode =
			policy->filenames_encryption_mode;
		ctx->flags = policy->flags;
		memcpy(ctx->master_key_descriptor,
		       policy->master_key_descriptor,
		       sizeof(ctx->master_key_descriptor));
		get_random_bytes(ctx->nonce, sizeof(ctx->nonce));
		return sizeof(*ctx);
	}
	case FSCRYPT_POLICY_V2: {
		const struct fscrypt_policy_v2 *policy = &policy_u->v2;
		struct fscrypt_context_v2 *ctx = &ctx_u->v2;

		ctx->version = FSCRYPT_CONTEXT_V2;
		ctx->contents_encryption_mode =
			policy->contents_encryption_mode;
		ctx->filenames_encryption_mode =
			policy->filenames_encryption_mode;
		ctx->flags = policy->flags;
		memcpy(ctx->master_key_identifier,
		       policy->master_key_identifier,
		       sizeof(ctx->master_key_identifier));
		get_random_bytes(ctx->nonce, sizeof(ctx->nonce));
		return sizeof(*ctx);
	}
	}
	BUG();
}

/**
 * fscrypt_policy_from_context - convert an fscrypt_context to an fscrypt_policy
 *
 * Given an fscrypt_context, build the corresponding fscrypt_policy.
 *
 * Return: 0 on success, or -EINVAL if the fscrypt_context has an unrecognized
 * version number or size.
 *
 * This does *not* validate the settings within the policy itself, e.g. the
 * modes, flags, and reserved bits.  Use fscrypt_supported_policy() for that.
 */
int fscrypt_policy_from_context(union fscrypt_policy *policy_u,
				const union fscrypt_context *ctx_u,
				int ctx_size)
{
	memset(policy_u, 0, sizeof(*policy_u));

	if (ctx_size <= 0 || ctx_size != fscrypt_context_size(ctx_u))
		return -EINVAL;

	switch (ctx_u->version) {
	case FSCRYPT_CONTEXT_V1: {
		const struct fscrypt_context_v1 *ctx = &ctx_u->v1;
		struct fscrypt_policy_v1 *policy = &policy_u->v1;

		policy->version = FSCRYPT_POLICY_V1;
		policy->contents_encryption_mode =
			ctx->contents_encryption_mode;
		policy->filenames_encryption_mode =
			ctx->filenames_encryption_mode;
		policy->flags = ctx->flags;
		memcpy(policy->master_key_descriptor,
		       ctx->master_key_descriptor,
		       sizeof(policy->master_key_descriptor));
		return 0;
	}
	case FSCRYPT_CONTEXT_V2: {
		const struct fscrypt_context_v2 *ctx = &ctx_u->v2;
		struct fscrypt_policy_v2 *policy = &policy_u->v2;

		policy->version = FSCRYPT_POLICY_V2;
		policy->contents_encryption_mode =
			ctx->contents_encryption_mode;
		policy->filenames_encryption_mode =
			ctx->filenames_encryption_mode;
		policy->flags = ctx->flags;
		memcpy(policy->__reserved, ctx->__reserved,
		       sizeof(policy->__reserved));
		memcpy(policy->master_key_identifier,
		       ctx->master_key_identifier,
		       sizeof(policy->master_key_identifier));
		return 0;
	}
	}
	/* unreachable */
	return -EINVAL;
}

/* Retrieve an inode's encryption policy */
static int fscrypt_get_policy(struct inode *inode, union fscrypt_policy *policy)
{
	const struct fscrypt_info *ci;
	union fscrypt_context ctx;
	int ret;

	ci = READ_ONCE(inode->i_crypt_info);
	if (ci) {
		/* key available, use the cached policy */
		*policy = ci->ci_policy;
		return 0;
	}

	if (!IS_ENCRYPTED(inode))
		return -ENODATA;

	ret = inode->i_sb->s_cop->get_context(inode, &ctx, sizeof(ctx));
	if (ret < 0)
		return (ret == -ERANGE) ? -EINVAL : ret;

	return fscrypt_policy_from_context(policy, &ctx, ret);
}

static int set_encryption_policy(struct inode *inode,
				 const union fscrypt_policy *policy)
{
	union fscrypt_context ctx;
	int ctxsize;
	int err;

	if (!fscrypt_supported_policy(policy, inode))
		return -EINVAL;

	if (policy->flags & ~FS_POLICY_FLAGS_VALID)
		return -EINVAL;

	ctx.contents_encryption_mode = policy->contents_encryption_mode;
	ctx.filenames_encryption_mode = policy->filenames_encryption_mode;
	ctx.flags = policy->flags;
	BUILD_BUG_ON(sizeof(ctx.nonce) != FS_KEY_DERIVATION_NONCE_SIZE);
	get_random_bytes(ctx.nonce, FS_KEY_DERIVATION_NONCE_SIZE);

	return inode->i_sb->s_cop->set_context(inode, &ctx, sizeof(ctx), NULL);
}

int fscrypt_ioctl_set_policy(struct file *filp, const void __user *arg)
{
	struct fscrypt_policy policy;
	struct inode *inode = file_inode(filp);
	int ret;
	struct fscrypt_context ctx;

	if (copy_from_user(&policy, arg, sizeof(policy)))
		return -EFAULT;

	if (!inode_owner_or_capable(inode))
		return -EACCES;

	if (policy.version != 0)
		return -EINVAL;

	ret = mnt_want_write_file(filp);
	if (ret)
		return ret;

	inode_lock(inode);

	ret = inode->i_sb->s_cop->get_context(inode, &ctx, sizeof(ctx));
	if (ret == -ENODATA) {
		if (!S_ISDIR(inode->i_mode))
			ret = -ENOTDIR;
		else if (IS_DEADDIR(inode))
			ret = -ENOENT;
		else if (!inode->i_sb->s_cop->empty_dir(inode))
			ret = -ENOTEMPTY;
		else
			ret = create_encryption_context_from_policy(inode,
								    &policy);
	} else if (ret == sizeof(ctx) &&
		   is_encryption_context_consistent_with_policy(&ctx,
								&policy)) {
		/* The file already uses the same encryption policy. */
		ret = 0;
	} else if (ret >= 0 || ret == -ERANGE) {
		/* The file already uses a different encryption policy. */
		ret = -EEXIST;
	}

	inode_unlock(inode);

	mnt_drop_write_file(filp);
	return ret;
}
EXPORT_SYMBOL(fscrypt_ioctl_set_policy);

int fscrypt_ioctl_get_policy(struct file *filp, void __user *arg)
{
	struct inode *inode = file_inode(filp);
	struct fscrypt_context ctx;
	struct fscrypt_policy policy;
	int res;

	if (!IS_ENCRYPTED(inode))
		return -ENODATA;

	res = inode->i_sb->s_cop->get_context(inode, &ctx, sizeof(ctx));
	if (res < 0 && res != -ERANGE)
		return res;
	if (res != sizeof(ctx))
		return -EINVAL;
	if (ctx.format != FS_ENCRYPTION_CONTEXT_FORMAT_V1)
		return -EINVAL;

	policy.version = 0;
	policy.contents_encryption_mode = ctx.contents_encryption_mode;
	policy.filenames_encryption_mode = ctx.filenames_encryption_mode;
	policy.flags = ctx.flags;
	memcpy(policy.master_key_descriptor, ctx.master_key_descriptor,
				FS_KEY_DESCRIPTOR_SIZE);

	if (copy_to_user(arg, &policy, sizeof(policy)))
		return -EFAULT;
	return 0;
}
EXPORT_SYMBOL(fscrypt_ioctl_get_policy);

/**
 * fscrypt_has_permitted_context() - is a file's encryption policy permitted
 *				     within its directory?
 *
 * @parent: inode for parent directory
 * @child: inode for file being looked up, opened, or linked into @parent
 *
 * Filesystems must call this before permitting access to an inode in a
 * situation where the parent directory is encrypted (either before allowing
 * ->lookup() to succeed, or for a regular file before allowing it to be opened)
 * and before any operation that involves linking an inode into an encrypted
 * directory, including link, rename, and cross rename.  It enforces the
 * constraint that within a given encrypted directory tree, all files use the
 * same encryption policy.  The pre-access check is needed to detect potentially
 * malicious offline violations of this constraint, while the link and rename
 * checks are needed to prevent online violations of this constraint.
 *
 * Return: 1 if permitted, 0 if forbidden.
 */
int fscrypt_has_permitted_context(struct inode *parent, struct inode *child)
{
	const struct fscrypt_operations *cops = parent->i_sb->s_cop;
	const struct fscrypt_info *parent_ci, *child_ci;
	struct fscrypt_context parent_ctx, child_ctx;
	int res;

	/* No restrictions on file types which are never encrypted */
	if (!S_ISREG(child->i_mode) && !S_ISDIR(child->i_mode) &&
	    !S_ISLNK(child->i_mode))
		return 1;

	/* No restrictions if the parent directory is unencrypted */
	if (!IS_ENCRYPTED(parent))
		return 1;

	/* Encrypted directories must not contain unencrypted files */
	if (!IS_ENCRYPTED(child))
		return 0;

	/*
	 * Both parent and child are encrypted, so verify they use the same
	 * encryption policy.  Compare the fscrypt_info structs if the keys are
	 * available, otherwise retrieve and compare the fscrypt_contexts.
	 *
	 * Note that the fscrypt_context retrieval will be required frequently
	 * when accessing an encrypted directory tree without the key.
	 * Performance-wise this is not a big deal because we already don't
	 * really optimize for file access without the key (to the extent that
	 * such access is even possible), given that any attempted access
	 * already causes a fscrypt_context retrieval and keyring search.
	 *
	 * In any case, if an unexpected error occurs, fall back to "forbidden".
	 */

	res = fscrypt_get_encryption_info(parent);
	if (res)
		return 0;
	res = fscrypt_get_encryption_info(child);
	if (res)
		return 0;
	parent_ci = READ_ONCE(parent->i_crypt_info);
	child_ci = READ_ONCE(child->i_crypt_info);

	if (parent_ci && child_ci) {
		return memcmp(parent_ci->ci_master_key_descriptor,
			      child_ci->ci_master_key_descriptor,
			      FS_KEY_DESCRIPTOR_SIZE) == 0 &&
			(parent_ci->ci_data_mode == child_ci->ci_data_mode) &&
			(parent_ci->ci_filename_mode ==
			 child_ci->ci_filename_mode) &&
			(parent_ci->ci_flags == child_ci->ci_flags);
	}

	res = cops->get_context(parent, &parent_ctx, sizeof(parent_ctx));
	if (res != sizeof(parent_ctx))
		return 0;

	res = cops->get_context(child, &child_ctx, sizeof(child_ctx));
	if (res != sizeof(child_ctx))
		return 0;

	return memcmp(parent_ctx.master_key_descriptor,
		      child_ctx.master_key_descriptor,
		      FS_KEY_DESCRIPTOR_SIZE) == 0 &&
		(parent_ctx.contents_encryption_mode ==
		 child_ctx.contents_encryption_mode) &&
		(parent_ctx.filenames_encryption_mode ==
		 child_ctx.filenames_encryption_mode) &&
		(parent_ctx.flags == child_ctx.flags);
}
EXPORT_SYMBOL(fscrypt_has_permitted_context);

/**
 * fscrypt_inherit_context() - Sets a child context from its parent
 * @parent: Parent inode from which the context is inherited.
 * @child:  Child inode that inherits the context from @parent.
 * @fs_data:  private data given by FS.
 * @preload:  preload child i_crypt_info if true
 *
 * Return: 0 on success, -errno on failure
 */
int fscrypt_inherit_context(struct inode *parent, struct inode *child,
						void *fs_data, bool preload)
{
	struct fscrypt_context ctx;
	struct fscrypt_info *ci;
	int res;

	res = fscrypt_get_encryption_info(parent);
	if (res < 0)
		return res;

	ci = READ_ONCE(parent->i_crypt_info);
	if (ci == NULL)
		return -ENOKEY;

	ctx.format = FS_ENCRYPTION_CONTEXT_FORMAT_V1;
	ctx.contents_encryption_mode = ci->ci_data_mode;
	ctx.filenames_encryption_mode = ci->ci_filename_mode;
	ctx.flags = ci->ci_flags;
	memcpy(ctx.master_key_descriptor, ci->ci_master_key_descriptor,
	       FS_KEY_DESCRIPTOR_SIZE);
	get_random_bytes(ctx.nonce, FS_KEY_DERIVATION_NONCE_SIZE);
	BUILD_BUG_ON(sizeof(ctx) != FSCRYPT_SET_CONTEXT_MAX_SIZE);
	res = parent->i_sb->s_cop->set_context(child, &ctx,
						sizeof(ctx), fs_data);
	if (res)
		return res;
	return preload ? fscrypt_get_encryption_info(child): 0;
}
EXPORT_SYMBOL(fscrypt_inherit_context);
